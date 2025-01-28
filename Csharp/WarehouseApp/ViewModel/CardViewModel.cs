﻿using System.Windows.Input;
using WarehouseApp.ViewModel;

public class CardViewModel : ViewModelBase
{
    private readonly WarehouseApp.Model.Component linkedComponent;

    public int Quantity 
    {
        get => linkedComponent.Quantity; 
        set
        {
            linkedComponent.Quantity = value;
            OnPropertyChanged();
        }
    }

    public string ImageSource => linkedComponent.ImageSource;

    public string Title => linkedComponent.Name;

    public CardViewModel(WarehouseApp.Model.Component linkedComponent)
    {
        this.linkedComponent = linkedComponent;

        PlusCommand = new Command(() => Quantity++);
        MinusCommand = new Command(() => 
        { 
            if (Quantity > 0) 
                Quantity--; 
        });
    }

    public ICommand PlusCommand { get; }

    public ICommand MinusCommand { get; }
}