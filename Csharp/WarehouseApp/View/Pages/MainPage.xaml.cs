﻿using WarehouseApp.ViewModel;

namespace WarehouseApp;

public partial class MainPage : ContentPage
{
    // Конструктор класса
    public MainPage()
    {
        InitializeComponent();
        BindingContext = new MainPageViewModel();
    }

    // Метод, срабатывающий при каждом заходе на страницу
    protected override void OnAppearing()
    {
        base.OnAppearing();

        if (BindingContext is MainPageViewModel viewModel)
        {
            // Обновление элементов страницы
            viewModel.RefreshItems();
        }
    }
}
