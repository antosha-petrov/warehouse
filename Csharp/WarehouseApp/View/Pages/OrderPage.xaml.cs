using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class OrderPage : ContentPage
{
    public OrderPage()
    {
        InitializeComponent();
        BindingContext = new OrderPageViewModel();
    }

    protected override void OnAppearing()
    {
        base.OnAppearing();

        if (BindingContext is OrderPageViewModel viewModel)
        {
            viewModel.RefreshItems();
        }
    }
}