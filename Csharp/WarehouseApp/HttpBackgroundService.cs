using System.Net.Http.Json;
using Models;

public class HttpBackgroundService
{
#pragma warning disable IDE0052
    private readonly Timer _timer;
#pragma warning restore IDE0052
    private readonly HttpClient _httpClient;
    private Order? _lastOrder;

    public HttpBackgroundService()
    {
        _httpClient = new HttpClient { BaseAddress = new Uri("http://Honor:5298/") };
        _timer = new Timer(async _ => await SendHttpRequest(), null, TimeSpan.Zero, TimeSpan.FromSeconds(2));
    }

    private async Task<Order?> SendHttpRequest()
    {
        HttpResponseMessage response = await _httpClient.GetAsync("orders/get/app");
        return _lastOrder = await response.Content.ReadFromJsonAsync<Order>();
    }
}